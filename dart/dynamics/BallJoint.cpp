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

#include "dart/dynamics/BallJoint.h"

#include <string>

#include "dart/math/Helpers.h"
#include "dart/math/Geometry.h"
#include "dart/dynamics/DegreeOfFreedom.h"

namespace dart {
namespace dynamics {

//==============================================================================
BallJoint::Properties::Properties(const Base::Properties& properties)
  : Base::Properties(properties)
{
  // Do nothing
}

//==============================================================================
BallJoint::~BallJoint()
{
  // Do nothing
}

//==============================================================================
const std::string& BallJoint::getType() const
{
  return getStaticType();
}

//==============================================================================
const std::string& BallJoint::getStaticType()
{
  static const std::string name = "BallJoint";
  return name;
}

//==============================================================================
bool BallJoint::isCyclic(std::size_t _index) const
{
  return _index < 3
      && !hasPositionLimit(0) && !hasPositionLimit(1) && !hasPositionLimit(2);
}

//==============================================================================
BallJoint::Properties BallJoint::getBallJointProperties() const
{
  return getGeometricJointProperties();
}

//==============================================================================
Eigen::Isometry3d BallJoint::convertToTransform(
    const Eigen::Vector3d& _positions)
{
  return Eigen::Isometry3d(convertToRotation(_positions));
}

//==============================================================================
Eigen::Matrix3d BallJoint::convertToRotation(const Eigen::Vector3d& _positions)
{
  return math::expMapRot(_positions);
}

//==============================================================================
BallJoint::BallJoint(const Properties& properties)
  : Base(properties),
    mR(Eigen::Isometry3d::Identity())
{
  mJacobianDeriv = Eigen::Matrix<double, 6, 3>::Zero();

  // Inherited Aspects must be created in the final joint class in reverse order
  // or else we get pure virtual function calls
  createGeometricJointAspect(properties);
  createJointAspect(properties);
}

//==============================================================================
Joint* BallJoint::clone() const
{
  return new BallJoint(getBallJointProperties());
}

//==============================================================================
const Eigen::Matrix<double, 6, 3> BallJoint::getLocalJacobianStatic(
    const Eigen::Vector3d& /*positions*/) const
{
  return mJacobian;
}

//==============================================================================
Eigen::Vector3d BallJoint::getPositionDifferencesStatic(
    const Eigen::Vector3d& _q2, const Eigen::Vector3d& _q1) const
{
  const Eigen::Matrix3d R1 = convertToRotation(_q1);
  const Eigen::Matrix3d R2 = convertToRotation(_q2);

  return convertToPositions(R1.transpose() * R2);
}

//==============================================================================
void BallJoint::integratePositions(double _dt)
{
  Eigen::Matrix3d Rnext
      = getR().linear() * convertToRotation(getVelocitiesStatic() * _dt);

  setPositionsStatic(convertToPositions(Rnext));
}

//==============================================================================
void BallJoint::updateDegreeOfFreedomNames()
{
  if(!mDofs[0]->isNamePreserved())
    mDofs[0]->setName(Joint::mAspectProperties.mName + "_x", false);
  if(!mDofs[1]->isNamePreserved())
    mDofs[1]->setName(Joint::mAspectProperties.mName + "_y", false);
  if(!mDofs[2]->isNamePreserved())
    mDofs[2]->setName(Joint::mAspectProperties.mName + "_z", false);
}

//==============================================================================
void BallJoint::updateLocalTransform() const
{
  mR.linear() = convertToRotation(getPositionsStatic());

  mT = Joint::mAspectProperties.mT_ParentBodyToJoint * mR
      * Joint::mAspectProperties.mT_ChildBodyToJoint.inverse();

  assert(math::verifyTransform(mT));
}

//==============================================================================
void BallJoint::updateLocalJacobian(bool _mandatory) const
{
  if (_mandatory)
  {
    mJacobian = math::getAdTMatrix(
          Joint::mAspectProperties.mT_ChildBodyToJoint).leftCols<3>();
  }
}

//==============================================================================
void BallJoint::updateLocalJacobianTimeDeriv() const
{
  assert(Eigen::Matrix6d::Zero().leftCols<3>() == mJacobianDeriv);
}

//==============================================================================
const Eigen::Isometry3d& BallJoint::getR() const
{
  if(mNeedTransformUpdate)
  {
    updateLocalTransform();
    mNeedTransformUpdate = false;
  }

  return mR;
}

}  // namespace dynamics
}  // namespace dart

