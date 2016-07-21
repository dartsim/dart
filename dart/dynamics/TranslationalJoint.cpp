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

#include "dart/dynamics/TranslationalJoint.hpp"

#include <string>

#include "dart/math/Geometry.hpp"
#include "dart/math/Helpers.hpp"

namespace dart {
namespace dynamics {

//==============================================================================
TranslationalJoint::Properties::Properties(
    const Base::Properties& _properties)
  : Base::Properties(_properties)
{
  // Do nothing
}

//==============================================================================
TranslationalJoint::~TranslationalJoint()
{
  // Do nothing
}

//==============================================================================
Eigen::Matrix<double, 6, 3> TranslationalJoint::getRelativeJacobianStatic(
    const Eigen::Vector3d& /*_positions*/) const
{
  // The Jacobian is always constant w.r.t. the generalized coordinates.
  return getRelativeJacobianStatic();
}

//==============================================================================
TranslationalJoint::Properties
TranslationalJoint::getTranslationalJointProperties() const
{
  return getGenericJointProperties();
}

//==============================================================================
TranslationalJoint::TranslationalJoint(const Properties& properties)
  : Base(properties)
{
  // Inherited Aspects must be created in the final joint class in reverse order
  // or else we get pure virtual function calls
  createGenericJointAspect(properties);
  createJointAspect(properties);
}

//==============================================================================
Joint* TranslationalJoint::clone() const
{
  return new TranslationalJoint(getTranslationalJointProperties());
}

//==============================================================================
const std::string& TranslationalJoint::getType() const
{
  return getStaticType();
}

//==============================================================================
const std::string& TranslationalJoint::getStaticType()
{
  static const std::string name = "TranslationalJoint";
  return name;
}

//==============================================================================
bool TranslationalJoint::isCyclic(std::size_t /*_index*/) const
{
  return false;
}

//==============================================================================
void TranslationalJoint::updateDegreeOfFreedomNames()
{
  if(!mDofs[0]->isNamePreserved())
    mDofs[0]->setName(Joint::mAspectProperties.mName + "_x", false);
  if(!mDofs[1]->isNamePreserved())
    mDofs[1]->setName(Joint::mAspectProperties.mName + "_y", false);
  if(!mDofs[2]->isNamePreserved())
    mDofs[2]->setName(Joint::mAspectProperties.mName + "_z", false);
}

//==============================================================================
void TranslationalJoint::updateRelativeTransform() const
{
  mT = Joint::mAspectProperties.mT_ParentBodyToJoint
       * Eigen::Translation3d(getPositionsStatic())
       * Joint::mAspectProperties.mT_ChildBodyToJoint.inverse();

  // Verification
  assert(math::verifyTransform(mT));
}

//==============================================================================
void TranslationalJoint::updateRelativeJacobian(bool _mandatory) const
{
  if (_mandatory)
  {
    mJacobian.bottomRows<3>() = Joint::mAspectProperties.mT_ChildBodyToJoint.linear();

    // Verification
    assert(mJacobian.topRows<3>() == Eigen::Matrix3d::Zero());
    assert(!math::isNan(mJacobian.bottomRows<3>()));
  }
}

//==============================================================================
void TranslationalJoint::updateRelativeJacobianTimeDeriv() const
{
  // Time derivative of translational joint is always zero
  assert(mJacobianDeriv == (Eigen::Matrix<double, 6, 3>::Zero()));
}

}  // namespace dynamics
}  // namespace dart
