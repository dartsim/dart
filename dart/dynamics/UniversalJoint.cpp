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

#include "kido/dynamics/UniversalJoint.h"

#include <string>

#include "kido/math/Helpers.h"
#include "kido/math/Geometry.h"

namespace kido {
namespace dynamics {

//==============================================================================
UniversalJoint::UniqueProperties::UniqueProperties(
    const Eigen::Vector3d& _axis1, const Eigen::Vector3d& _axis2)
  : mAxis({_axis1.normalized(), _axis2.normalized()})
{
  // Do nothing
}

//==============================================================================
UniversalJoint::Properties::Properties(
    const MultiDofJoint::Properties& _multiDofProperties,
    const UniversalJoint::UniqueProperties& _universalProperties)
  : MultiDofJoint::Properties(_multiDofProperties),
    UniversalJoint::UniqueProperties(_universalProperties)
{
  // Do nothing
}

//==============================================================================
UniversalJoint::~UniversalJoint()
{
  // Do nothing
}

//==============================================================================
void UniversalJoint::setProperties(const Properties& _properties)
{
  MultiDofJoint<2>::setProperties(
        static_cast<const MultiDofJoint<2>::Properties&>(_properties));
  setProperties(static_cast<const UniqueProperties&>(_properties));
}

//==============================================================================
void UniversalJoint::setProperties(const UniqueProperties& _properties)
{
  setAxis1(_properties.mAxis[0]);
  setAxis2(_properties.mAxis[1]);
}

//==============================================================================
UniversalJoint::Properties UniversalJoint::getUniversalJointProperties() const
{
  return Properties(getMultiDofJointProperties(), mUniversalP);
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
bool UniversalJoint::isCyclic(size_t _index) const
{
  return !hasPositionLimit(_index);
}

//==============================================================================
void UniversalJoint::setAxis1(const Eigen::Vector3d& _axis)
{
  mUniversalP.mAxis[0] = _axis.normalized();
  notifyPositionUpdate();
}

//==============================================================================
void UniversalJoint::setAxis2(const Eigen::Vector3d& _axis)
{
  mUniversalP.mAxis[1] = _axis.normalized();
  notifyPositionUpdate();
}

//==============================================================================
const Eigen::Vector3d& UniversalJoint::getAxis1() const
{
  return mUniversalP.mAxis[0];
}

//==============================================================================
const Eigen::Vector3d& UniversalJoint::getAxis2() const
{
  return mUniversalP.mAxis[1];
}

//==============================================================================
Eigen::Matrix<double, 6, 2> UniversalJoint::getLocalJacobianStatic(
    const Eigen::Vector2d& _positions) const
{
  Eigen::Matrix<double, 6, 2> J;
  J.col(0) = math::AdTAngular(
               mJointP.mT_ChildBodyToJoint
               * math::expAngular(-mUniversalP.mAxis[1] * _positions[1]),
                                  mUniversalP.mAxis[0]);
  J.col(1) = math::AdTAngular(mJointP.mT_ChildBodyToJoint,
                              mUniversalP.mAxis[1]);
  assert(!math::isNan(J));
  return J;
}

//==============================================================================
UniversalJoint::UniversalJoint(const Properties& _properties)
  : MultiDofJoint<2>(_properties)
{
  setProperties(_properties);
  updateDegreeOfFreedomNames();
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
    mDofs[0]->setName(mJointP.mName + "_1", false);
  if(!mDofs[1]->isNamePreserved())
    mDofs[1]->setName(mJointP.mName + "_2", false);
}

//==============================================================================
void UniversalJoint::updateLocalTransform() const
{
  const Eigen::Vector2d& positions = getPositionsStatic();
  mT = mJointP.mT_ParentBodyToJoint
       * Eigen::AngleAxisd(positions[0], mUniversalP.mAxis[0])
       * Eigen::AngleAxisd(positions[1], mUniversalP.mAxis[1])
       * mJointP.mT_ChildBodyToJoint.inverse();
  assert(math::verifyTransform(mT));
}

//==============================================================================
void UniversalJoint::updateLocalJacobian(bool) const
{
  mJacobian = getLocalJacobianStatic(getPositionsStatic());
}

//==============================================================================
void UniversalJoint::updateLocalJacobianTimeDeriv() const
{
  Eigen::Vector6d tmpV1 = getLocalJacobianStatic().col(1)
                        * getVelocitiesStatic()[1];

  Eigen::Isometry3d tmpT = math::expAngular(-mUniversalP.mAxis[1]
                                            * getPositionsStatic()[1]);

  Eigen::Vector6d tmpV2
      = math::AdTAngular(mJointP.mT_ChildBodyToJoint * tmpT,
                         mUniversalP.mAxis[0]);

  mJacobianDeriv.col(0) = -math::ad(tmpV1, tmpV2);

  assert(!math::isNan(mJacobianDeriv.col(0)));
  assert(mJacobianDeriv.col(1) == Eigen::Vector6d::Zero());
}

}  // namespace dynamics
}  // namespace kido
